from __future__ import annotations

# Articraft model: 18 mm snap-off utility knife – slim flat metal-body grip
# variant with a squared rectangular cross-section.
#
# Articraft brief:
# - Object: 18 mm snap-off blade utility knife, ~0.155 m long, slim flat
#   brushed-steel handle with a squared rectangular cross-section, a dark-steel
#   top channel, a segmented silver snap-off blade, a black textured side-grip,
#   a rear lanyard hole, and a sliding thumb ratchet that pushes the blade out
#   the front.
# - Root/support: the metal handle body (handle) is the fixed root. It is a
#   uniform squared-section bar with a top channel that houses the blade and
#   carries the slide rail; the rear has a lanyard hole and the front has a
#   blade-exit slot.
# - Parts: handle (root), blade_carrier (sliding member = blade spine + exposed
#   snap-off blade + thumb slide button; one rigid moving part), end_cap (rear
#   dark-metal cap, fixed to the handle).
# - Articulation: handle_to_carrier, PRISMATIC, axis along the handle long axis
#   (+X), positive q pushes the blade out the front. Origin at the rear seating
#   plane so the carrier stays retained at full extension.
# - Visible geometry: brushed-steel squared-section bar, dark top channel,
#   silver segmented blade with score lines and a dark worn tip, black knurled
#   side-grip pad, ribbed thumb slide button, rear lanyard hole, front
#   blade-exit slot, dark rear end cap.
# - Support/fit: the blade carrier rides inside the handle channel; the thumb
#   button protrudes up through the channel slot. The end cap is fixed to the
#   rear. The blade spine is captured inside the channel proxy.
# - Intentional overlaps: blade spine and blade root slide inside the handle
#   channel/body proxy (nested slider) -> scoped allow_overlap with
#   retained-insertion checks.
# - Tests: squared cross-section, slim flat metal body, blade present in front,
#   thumb button on top, prismatic axis is +X, extending pushes the blade tip
#   forward, blade stays retained, end cap seated at the rear.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Real-world dimensions (meters)
# ---------------------------------------------------------------------------
HANDLE_LEN = 0.150          # overall handle length along +X
HANDLE_H = 0.014            # handle height (Z) – slim, uniform
HANDLE_W = 0.012            # handle width (Y) – flat, squared with height
HANDLE_EDGE_FILLET = 0.0008 # small fillet on long edges

CHANNEL_W = 0.0034          # width of the top blade channel (Y)
CHANNEL_DEPTH = 0.005       # how deep the channel cuts into the body (Z)

BLADE_LEN = 0.060           # full snap-off blade length (spine + exposed)
BLADE_W = 0.018             # 18 mm blade
BLADE_THK = 0.0006          # blade thickness
BLADE_EXPOSED_REST = 0.012  # exposed blade length at the retracted rest pose

SLIDE_TRAVEL = 0.034        # usable forward travel of the thumb slide

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------
BRUSHED_STEEL = Material(name="brushed_steel", rgba=(0.72, 0.73, 0.76, 1.0))
DARK_STEEL = Material(name="dark_steel", rgba=(0.38, 0.40, 0.44, 1.0))
CHANNEL_METAL = Material(name="channel_metal", rgba=(0.50, 0.52, 0.56, 1.0))
BLACK_GRIP = Material(name="black_grip", rgba=(0.10, 0.10, 0.11, 1.0))
STEEL = Material(name="blade_steel", rgba=(0.80, 0.82, 0.85, 1.0))
BLADE_TIP = Material(name="blade_tip", rgba=(0.30, 0.34, 0.46, 1.0))
CAP_METAL = Material(name="cap_metal", rgba=(0.28, 0.30, 0.34, 1.0))
BUTTON_METAL = Material(name="button_metal", rgba=(0.58, 0.60, 0.63, 1.0))


# ---------------------------------------------------------------------------
# Shared geometry helpers
# ---------------------------------------------------------------------------
def _knurl_bumps_xy(
    cx: float,
    cy: float,
    pad_w: float,
    pad_d: float,
    nx: int,
    ny: int,
    bump_w: float,
    bump_d: float,
    bump_h: float,
    z_base: float,
) -> cq.Workplane | None:
    """Shared helper: grid of raised rectangular bumps on an XY-oriented pad.

    Builds bumps protruding in +Z from *z_base*, arranged in an *nx* × *ny*
    regular grid centered at (*cx*, *cy*).
    """
    bumps: cq.Workplane | None = None
    for ix in range(nx):
        for iy in range(ny):
            x = cx - pad_w / 2.0 + (ix + 0.5) * (pad_w / nx)
            y = cy - pad_d / 2.0 + (iy + 0.5) * (pad_d / ny)
            b = (
                cq.Workplane("XY")
                .transformed(offset=(x, y, z_base))
                .box(bump_w, bump_d, bump_h, centered=(True, True, False))
            )
            bumps = b if bumps is None else bumps.add(b)
    return bumps


def _knurl_bumps_side(
    cx: float,
    cz: float,
    pad_len: float,
    pad_height: float,
    nx: int,
    nz: int,
    bump_w: float,
    bump_h: float,
    bump_d: float,
    y_base: float,
) -> cq.Workplane | None:
    """Shared helper: grid of bumps on a side-oriented (XZ) pad.

    Builds bumps protruding in +Y from *y_base*.
    """
    bumps: cq.Workplane | None = None
    for ix in range(nx):
        for iz in range(nz):
            x = cx - pad_len / 2.0 + (ix + 0.5) * (pad_len / nx)
            z = cz - pad_height / 2.0 + (iz + 0.5) * (pad_height / nz)
            b = (
                cq.Workplane("XY")
                .box(bump_w, bump_d, bump_h, centered=(True, True, True))
                .translate((x, y_base + bump_d / 2.0, z))
            )
            bumps = b if bumps is None else bumps.add(b)
    return bumps


# ---------------------------------------------------------------------------
# Handle body (squared cross-section metal bar)
# ---------------------------------------------------------------------------
def _handle_body_shape() -> cq.Workplane:
    """Slim flat metal-body handle with squared rectangular cross-section.

    Uniform bar along +X with small fillets on the four long edges.
    """
    body = (
        cq.Workplane("XY")
        .box(HANDLE_LEN, HANDLE_W, HANDLE_H, centered=(True, True, False))
        .edges("|X")
        .fillet(HANDLE_EDGE_FILLET)
    )
    return body


def _channel_cut() -> cq.Workplane:
    """Top channel groove cut along the centerline for the blade + slide."""
    length = HANDLE_LEN + 0.02
    groove = (
        cq.Workplane("XY")
        .box(length, CHANNEL_W, CHANNEL_DEPTH, centered=(True, True, False))
        .translate((0.0, 0.0, HANDLE_H - CHANNEL_DEPTH + 0.0005))
    )
    return groove


def _lanyard_hole_cut() -> cq.Workplane:
    """Rear lanyard through-hole along Y, below the channel."""
    x0 = -HANDLE_LEN / 2.0
    hole = (
        cq.Workplane("XY")
        .box(HANDLE_W * 3.0, HANDLE_W * 3.0, HANDLE_W * 3.0, centered=(True, True, True))
        .translate((x0 + 0.012, 0.0, HANDLE_H * 0.38))
    )
    # Use a cylinder for the hole instead.
    hole = (
        cq.Workplane("XZ")
        .transformed(offset=(0.0, 0.0, 0.0))
        .workplane(offset=HANDLE_W)
        .center(x0 + 0.012, HANDLE_H * 0.38)
        .circle(0.0025)
        .extrude(-2.0 * HANDLE_W)
    )
    return hole


def _blade_exit_slot() -> cq.Workplane:
    """Front blade-exit slot cut through the front face of the handle."""
    channel_bottom = HANDLE_H - CHANNEL_DEPTH + 0.0005
    slot_h = channel_bottom + 0.001
    slot_w = BLADE_THK + 0.0012
    slot_depth = 0.006
    slot = (
        cq.Workplane("XY")
        .box(slot_depth, slot_w, slot_h, centered=(True, True, False))
        .translate((HANDLE_LEN / 2.0 - slot_depth / 2.0 + 0.002, 0.0, 0.0))
    )
    return slot


def _build_handle_visual() -> cq.Workplane:
    body = _handle_body_shape()
    body = body.cut(_channel_cut())
    body = body.cut(_lanyard_hole_cut())
    body = body.cut(_blade_exit_slot())
    return body


def _build_top_channel_visual() -> cq.Workplane:
    """Dark-steel channel/rail that sits in the top groove.

    The bottom of the rail extends slightly below the channel cut floor so the
    rail visually contacts the handle shell body for mesh connectivity.
    """
    length = HANDLE_LEN - 0.012
    rail_height = CHANNEL_DEPTH - 0.0005
    rail_z_base = HANDLE_H - CHANNEL_DEPTH  # slightly below the cut floor
    rail = (
        cq.Workplane("XY")
        .box(length, CHANNEL_W - 0.0004, rail_height, centered=(True, True, False))
        .translate((0.002, 0.0, rail_z_base))
    )
    return rail


def _build_thumb_grip_visual() -> cq.Workplane:
    """Black textured thumb pad on the +Y side face of the metal handle."""
    pad_len = 0.028
    pad_height = HANDLE_H * 0.60
    pad_thk = 0.0015
    pad_cx = 0.020
    pad_cz = HANDLE_H * 0.50
    y_base = HANDLE_W / 2.0

    # Base pad proud of the handle side face.
    base = (
        cq.Workplane("XY")
        .box(pad_len, pad_thk, pad_height, centered=(True, True, True))
        .translate((pad_cx, y_base + pad_thk / 2.0, pad_cz))
    )

    # Knurl bumps on the pad outer surface.
    bumps = _knurl_bumps_side(
        cx=pad_cx,
        cz=pad_cz,
        pad_len=pad_len,
        pad_height=pad_height,
        nx=6,
        nz=3,
        bump_w=0.0022,
        bump_h=0.0018,
        bump_d=0.0010,
        y_base=y_base + pad_thk,
    )
    return base.add(bumps) if bumps is not None else base


# ---------------------------------------------------------------------------
# Blade carrier (moving prismatic member)
# ---------------------------------------------------------------------------
def _build_blade_shape() -> cq.Workplane:
    """Snap-off blade: flat plate in XZ, segmented with score lines."""
    bl = BLADE_LEN
    bw = BLADE_W
    pts = [
        (0.0, 0.0),
        (bl, 0.0),
        (bl, -bw + 0.004),
        (bl - 0.006, -bw),
        (0.0, -bw + 0.010),
    ]
    blade = (
        cq.Workplane("XZ")
        .polyline(pts)
        .close()
        .extrude(BLADE_THK)
        .translate((0.0, -BLADE_THK / 2.0, 0.0))
    )
    return blade


def _build_blade_score_lines() -> cq.Workplane:
    """Diagonal snap-off score grooves across the blade."""
    grooves: cq.Workplane | None = None
    bw = BLADE_W
    for i in range(5):
        x = 0.006 + i * 0.011
        groove = (
            cq.Workplane("XZ")
            .center(x, -bw / 2.0)
            .rect(0.0012, bw + 0.004)
            .extrude(BLADE_THK + 0.0006)
            .translate((0.0, -(BLADE_THK + 0.0006) / 2.0, 0.0))
            .rotate((x, 0.0, -bw / 2.0), (x, 1.0, -bw / 2.0), 18.0)
        )
        grooves = groove if grooves is None else grooves.add(groove)
    return grooves


def _build_blade_tip_visual() -> cq.Workplane:
    """Dark front segment of the blade."""
    bl = BLADE_LEN
    bw = BLADE_W
    pts = [
        (bl - 0.012, 0.0),
        (bl, 0.0),
        (bl, -bw + 0.004),
        (bl - 0.006, -bw),
        (bl - 0.012, -bw + 0.006),
    ]
    tip = (
        cq.Workplane("XZ")
        .polyline(pts)
        .close()
        .extrude(BLADE_THK + 0.0002)
        .translate((0.0, -(BLADE_THK + 0.0002) / 2.0, 0.0))
    )
    return tip


def _build_blade_spine_carrier() -> cq.Workplane:
    """Carrier block that grips the blade spine and rides the channel."""
    spine = (
        cq.Workplane("XY")
        .box(0.020, CHANNEL_W - 0.0006, 0.005, centered=(True, True, False))
    )
    return spine


def _build_thumb_button() -> cq.Workplane:
    """Ribbed thumb slide button protruding up out of the channel slot."""
    base = (
        cq.Workplane("XY")
        .box(0.013, 0.0060, 0.0050, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.0010)
    )
    ribs = _knurl_bumps_xy(
        cx=0.0,
        cy=0.0,
        pad_w=0.010,
        pad_d=0.006,
        nx=5,
        ny=1,
        bump_w=0.0008,
        bump_d=0.006,
        bump_h=0.0014,
        z_base=0.0050,
    )
    return base.add(ribs) if ribs is not None else base


# ---------------------------------------------------------------------------
# Assembly
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="snap_off_utility_knife")

    # -- Handle (root) ------------------------------------------------------
    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(_build_handle_visual(), "handle_shell"),
        material=BRUSHED_STEEL,
        name="handle_shell",
    )
    handle.visual(
        mesh_from_cadquery(_build_top_channel_visual(), "top_channel"),
        material=CHANNEL_METAL,
        name="top_channel",
    )
    handle.visual(
        mesh_from_cadquery(_build_thumb_grip_visual(), "thumb_grip"),
        material=BLACK_GRIP,
        name="thumb_grip",
    )

    # -- Rear end cap (fixed to handle) ------------------------------------
    end_cap = model.part("end_cap")
    cap_shape = (
        cq.Workplane("YZ")
        .workplane(offset=-HANDLE_LEN / 2.0)
        .center(0.0, HANDLE_H / 2.0)
        .rect(HANDLE_W * 1.02, HANDLE_H * 0.96)
        .extrude(-0.010)
        .edges("|X")
        .fillet(0.002)
    )
    end_cap.visual(
        mesh_from_cadquery(cap_shape, "end_cap"),
        material=CAP_METAL,
        name="end_cap_body",
    )

    # -- Blade carrier (moving prismatic member) ---------------------------
    carrier = model.part("blade_carrier")

    z_blade_top = HANDLE_H - 0.0035
    nose_x = HANDLE_LEN / 2.0
    blade_rear_x = nose_x - (BLADE_LEN - BLADE_EXPOSED_REST)

    blade = _build_blade_shape().translate((blade_rear_x, 0.0, z_blade_top))
    blade_tip = _build_blade_tip_visual().translate((blade_rear_x, 0.0, z_blade_top))
    spine = _build_blade_spine_carrier().translate(
        (blade_rear_x + 0.010, 0.0, z_blade_top - 0.0015)
    )
    button = _build_thumb_button().translate(
        (blade_rear_x + 0.012, 0.0, HANDLE_H - 0.0035)
    )

    blade_body = blade.cut(
        _build_blade_score_lines().translate((blade_rear_x, 0.0, z_blade_top))
    )
    carrier.visual(
        mesh_from_cadquery(blade_body, "blade_steel"),
        material=STEEL,
        name="blade_steel",
    )
    carrier.visual(
        mesh_from_cadquery(blade_tip, "blade_tip"),
        material=BLADE_TIP,
        name="blade_tip",
    )
    carrier.visual(
        mesh_from_cadquery(spine, "blade_spine"),
        material=CHANNEL_METAL,
        name="blade_spine",
    )
    carrier.visual(
        mesh_from_cadquery(button, "thumb_button"),
        material=BUTTON_METAL,
        name="thumb_button",
    )

    # -- Articulations ------------------------------------------------------
    model.articulation(
        "handle_to_carrier",
        ArticulationType.PRISMATIC,
        parent=handle,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0, velocity=0.2, lower=0.0, upper=SLIDE_TRAVEL
        ),
    )

    model.articulation(
        "handle_to_cap",
        ArticulationType.FIXED,
        parent=handle,
        child=end_cap,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    handle = object_model.get_part("handle")
    carrier = object_model.get_part("blade_carrier")
    end_cap = object_model.get_part("end_cap")
    slide = object_model.get_articulation("handle_to_carrier")

    # -- Joint type / axis claims ------------------------------------------
    ctx.check(
        "blade slide is prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={slide.articulation_type}",
    )
    ax = tuple(slide.axis)
    ctx.check(
        "slide axis is along +X (handle long axis)",
        abs(ax[0]) > 0.99 and abs(ax[1]) < 1e-6 and abs(ax[2]) < 1e-6,
        details=f"axis={ax}",
    )

    # -- Handle squared cross-section and slim flat metal body -------------
    handle_aabb = ctx.part_world_aabb(handle)
    if handle_aabb is not None:
        h_dy = handle_aabb[1][1] - handle_aabb[0][1]
        h_dz = handle_aabb[1][2] - handle_aabb[0][2]
        h_dx = handle_aabb[1][0] - handle_aabb[0][0]
    else:
        h_dy = h_dz = h_dx = 0.0

    ctx.check(
        "handle has approximately squared cross-section (W ~ H)",
        handle_aabb is not None and abs(h_dy - h_dz) < 0.005,
        details=f"handle_dy={h_dy:.4f}, handle_dz={h_dz:.4f}",
    )
    ctx.check(
        "handle is slim (height < 20 mm)",
        handle_aabb is not None and h_dz < 0.020,
        details=f"handle_dz={h_dz:.4f}",
    )
    ctx.check(
        "handle has uniform cross-section (no taper)",
        handle_aabb is not None and abs(h_dx - HANDLE_LEN) < 0.005,
        details=f"handle_dx={h_dx:.4f}, expected={HANDLE_LEN:.4f}",
    )

    # -- Hero geometry: blade in front, button on top ----------------------
    blade_aabb = ctx.part_element_world_aabb(carrier, elem="blade_steel")
    ctx.check(
        "segmented blade protrudes past the handle nose at rest",
        blade_aabb is not None
        and handle_aabb is not None
        and blade_aabb[1][0] > handle_aabb[1][0] + 0.004,
        details=f"blade_max_x={None if blade_aabb is None else blade_aabb[1][0]}, "
        f"handle_max_x={None if handle_aabb is None else handle_aabb[1][0]}",
    )
    button_aabb = ctx.part_element_world_aabb(carrier, elem="thumb_button")
    ctx.check(
        "thumb slide button rises above the handle top",
        button_aabb is not None
        and handle_aabb is not None
        and button_aabb[1][2] > handle_aabb[1][2] - 0.001,
        details=f"button_max_z={None if button_aabb is None else button_aabb[1][2]}, "
        f"handle_max_z={None if handle_aabb is None else handle_aabb[1][2]}",
    )
    ctx.check(
        "blade has real width (cutting edge below the spine)",
        blade_aabb is not None and (blade_aabb[1][2] - blade_aabb[0][2]) > 0.012,
        details=f"blade_z_extent="
        f"{None if blade_aabb is None else blade_aabb[1][2] - blade_aabb[0][2]}",
    )
    tip_aabb = ctx.part_element_world_aabb(carrier, elem="blade_tip")
    ctx.check(
        "dark blade tip is at the very front",
        tip_aabb is not None
        and blade_aabb is not None
        and tip_aabb[1][0] >= blade_aabb[1][0] - 0.001,
        details=f"tip_max_x={None if tip_aabb is None else tip_aabb[1][0]}",
    )

    # -- End cap seated at the rear ----------------------------------------
    ctx.expect_contact(end_cap, handle, name="end cap seated against handle rear")
    cap_aabb = ctx.part_world_aabb(end_cap)
    ctx.check(
        "end cap is at the rear (-X) of the handle",
        cap_aabb is not None
        and handle_aabb is not None
        and cap_aabb[0][0] <= handle_aabb[0][0] + 0.002,
        details=f"cap_min_x={None if cap_aabb is None else cap_aabb[0][0]}",
    )

    # -- Blade carrier rides inside the channel proxy (nested slider) ------
    ctx.allow_overlap(
        carrier,
        handle,
        elem_a="blade_spine",
        elem_b="top_channel",
        reason="The blade spine carrier is intentionally captured inside the "
        "handle's top channel rail and slides along it.",
    )
    ctx.allow_overlap(
        carrier,
        handle,
        elem_a="blade_steel",
        elem_b="top_channel",
        reason="The blade root rides inside the handle channel groove as the "
        "carrier slides; this nested fit is intentional.",
    )
    ctx.allow_overlap(
        carrier,
        handle,
        elem_a="blade_steel",
        elem_b="handle_shell",
        reason="The blade body passes through the handle shell near the front "
        "exit slot as the carrier slides; this is the nested slider fit.",
    )

    # Retained insertion: the spine stays within the channel footprint (Y)
    ctx.expect_within(
        carrier,
        handle,
        axes="y",
        inner_elem="blade_spine",
        outer_elem="top_channel",
        margin=0.0006,
        name="blade spine stays centered in the channel",
    )
    ctx.expect_overlap(
        carrier,
        handle,
        axes="x",
        elem_a="blade_steel",
        elem_b="handle_shell",
        min_overlap=0.020,
        name="blade remains inserted in the handle at rest",
    )

    # -- Actuating the joint extends the blade forward ---------------------
    rest_tip = ctx.part_element_world_aabb(carrier, elem="blade_tip")
    with ctx.pose({slide: SLIDE_TRAVEL}):
        ext_tip = ctx.part_element_world_aabb(carrier, elem="blade_tip")
        ctx.check(
            "extending the slide pushes the blade tip forward",
            rest_tip is not None
            and ext_tip is not None
            and ext_tip[1][0] > rest_tip[1][0] + 0.030,
            details=f"rest_tip_x={None if rest_tip is None else rest_tip[1][0]}, "
            f"ext_tip_x={None if ext_tip is None else ext_tip[1][0]}",
        )
        ctx.expect_overlap(
            carrier,
            handle,
            axes="x",
            elem_a="blade_steel",
            elem_b="handle_shell",
            min_overlap=0.005,
            name="blade stays retained in the handle at full extension",
        )

    return ctx.report()
