from __future__ import annotations

# Articraft model: 18 mm snap-off utility knife with rubber-overmolded barrel
# grip and raised TPR ribs around the body. Fork variant of the parent knife.
#
# Articraft brief:
# - Object: 18 mm snap-off blade utility knife, ~0.155 m long, rubber-
#   overmolded barrel grip with raised TPR ribs, gray channel/spine, segmented
#   silver snap-off blade, sliding thumb ratchet.
# - Root/support: the barrel-grip handle (handle) is the fixed root. It is a
#   lofted circular barrel with a top channel that houses the blade rail.
# - Parts: handle (root), blade_carrier (sliding member), end_cap (rear cap).
# - Articulation: handle_to_carrier, PRISMATIC, +X axis, positive q extends
#   the blade forward. Origin at rest position.
# - Visible geometry: dark rubber barrel body, raised TPR rib rings, gray top
#   channel rail, silver segmented blade with score lines and dark tip, ribbed
#   gray thumb slide button, rear end cap.
# - Support/fit: blade carrier rides inside the handle channel; thumb button
#   protrudes up through the channel slot.
# - Intentional overlaps: blade spine slides inside the handle channel proxy.
# - Tests: barrel has circular cross-section, TPR ribs present, blade slide
#   mechanism works, blade stays retained.

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
HANDLE_LEN = 0.150
BARREL_R_MAX = 0.0145          # max barrel radius (~29 mm diameter grip)
BARREL_CENTER_Z = 0.012        # barrel axis height above origin
HANDLE_H = BARREL_CENTER_Z + BARREL_R_MAX  # effective top ≈ 0.0265

CHANNEL_W = 0.0034
CHANNEL_DEPTH = 0.009

BLADE_LEN = 0.060
BLADE_W = 0.018
BLADE_THK = 0.0006
BLADE_EXPOSED_REST = 0.012
SLIDE_TRAVEL = 0.034

# TPR rib parameters
N_RIBS = 8
RIB_MINOR_R = 0.0006           # rib torus tube cross-section radius
RIB_ZONE_START = -0.050        # first rib X position
RIB_ZONE_END = 0.020           # last rib X position

# Barrel profile: (x_position, radius) — defines the barrel loft
BARREL_PROFILE = [
    (-0.075, 0.009),
    (-0.060, 0.012),
    (-0.045, 0.014),
    (-0.020, 0.0145),
    (0.010, 0.0145),
    (0.035, 0.014),
    (0.055, 0.011),
    (0.068, 0.007),
    (0.075, 0.004),
]

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------
BARREL_RUBBER = Material(name="barrel_rubber", rgba=(0.22, 0.22, 0.24, 1.0))
TPR_RUBBER = Material(name="tpr_rubber", rgba=(0.14, 0.14, 0.16, 1.0))
GRAY = Material(name="channel_gray", rgba=(0.62, 0.63, 0.66, 1.0))
DARK_GRAY = Material(name="dark_gray", rgba=(0.22, 0.23, 0.25, 1.0))
STEEL = Material(name="blade_steel", rgba=(0.80, 0.82, 0.85, 1.0))
BLADE_TIP = Material(name="blade_tip", rgba=(0.30, 0.34, 0.46, 1.0))


# ---------------------------------------------------------------------------
# Barrel-grip handle geometry
# ---------------------------------------------------------------------------
def _barrel_radius_at(x: float) -> float:
    """Linearly interpolate barrel radius at position x along the handle."""
    for i in range(len(BARREL_PROFILE) - 1):
        x0, r0 = BARREL_PROFILE[i]
        x1, r1 = BARREL_PROFILE[i + 1]
        if x0 <= x <= x1:
            t = (x - x0) / (x1 - x0)
            return r0 + t * (r1 - r0)
    if x < BARREL_PROFILE[0][0]:
        return BARREL_PROFILE[0][1]
    return BARREL_PROFILE[-1][1]


def _build_barrel_body() -> cq.Workplane:
    """Lofted circular barrel shell, axis along +X at Y=0, Z=BARREL_CENTER_Z."""
    wires = []
    for x, r in BARREL_PROFILE:
        w = (
            cq.Workplane("YZ")
            .workplane(offset=x)
            .center(0.0, BARREL_CENTER_Z)
            .circle(r)
            .val()
        )
        wires.append(w)
    solid = cq.Solid.makeLoft(wires, ruled=False)
    return cq.Workplane("XY").newObject([solid])


def _channel_cut() -> cq.Workplane:
    """Top channel groove cut along the barrel centerline for the blade rail."""
    length = HANDLE_LEN + 0.02
    groove = (
        cq.Workplane("XY")
        .box(length, CHANNEL_W, CHANNEL_DEPTH, centered=(True, True, False))
        .translate((0.0, 0.0, HANDLE_H - CHANNEL_DEPTH + 0.0005))
    )
    return groove


def _lanyard_hole_cut() -> cq.Workplane:
    """Rear finger/lanyard through-hole along Y through the barrel."""
    x_rear = -HANDLE_LEN / 2.0 + 0.014
    hole = (
        cq.Workplane("XZ")
        .workplane(offset=0.020)
        .center(x_rear, BARREL_CENTER_Z)
        .circle(0.0042)
        .extrude(-0.040)
    )
    return hole


def _build_handle_visual() -> cq.Workplane:
    """Barrel handle shell with channel groove and lanyard hole."""
    body = _build_barrel_body()
    body = body.cut(_channel_cut())
    body = body.cut(_lanyard_hole_cut())
    return body


def _build_top_channel_visual() -> cq.Workplane:
    """Gray metal channel/rail that sits in the top groove."""
    length = HANDLE_LEN - 0.012
    rail = (
        cq.Workplane("XY")
        .box(length, CHANNEL_W - 0.0004, CHANNEL_DEPTH - 0.001,
             centered=(True, True, False))
        .translate((0.002, 0.0, HANDLE_H - CHANNEL_DEPTH + 0.0008))
    )
    return rail


def _build_tpr_rib(x_pos: float) -> cq.Workplane:
    """Single raised TPR rib: a torus ring revolved around the barrel axis.

    The rib cross-section circle is placed in the XZ plane above the barrel
    axis, then revolved 360° around the barrel centerline to create a full
    circumferential ring proud of the barrel surface.
    """
    barrel_r = _barrel_radius_at(x_pos)
    # Tube center slightly inward from barrel surface for visual embed/connection
    tube_center_r = barrel_r + RIB_MINOR_R * 0.7
    rib = (
        cq.Workplane("XZ")
        .moveTo(x_pos, BARREL_CENTER_Z + tube_center_r)
        .circle(RIB_MINOR_R)
        .revolve(360, (-0.1, BARREL_CENTER_Z), (0.1, BARREL_CENTER_Z))
    )
    return rib


# ---------------------------------------------------------------------------
# Blade carrier geometry (unchanged from parent)
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
    grooves = None
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
    """Dark front segment of the blade (worn / coated tip)."""
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
    """Ribbed gray thumb slide button protruding up out of the channel slot."""
    base = (
        cq.Workplane("XY")
        .box(0.013, 0.0060, 0.0050, centered=(True, True, False))
        .edges("|Z").fillet(0.0010)
    )
    ribs = None
    for i in range(5):
        x = -0.004 + i * 0.002
        r = (
            cq.Workplane("XY")
            .transformed(offset=(x, 0.0, 0.0050))
            .box(0.0008, 0.0060, 0.0014, centered=(True, True, False))
        )
        ribs = r if ribs is None else ribs.add(r)
    return base.add(ribs)


# ---------------------------------------------------------------------------
# Model assembly
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="snap_off_utility_knife")

    # --- Handle (root): barrel grip with TPR ribs --------------------------
    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(_build_handle_visual(), "barrel_shell"),
        material=BARREL_RUBBER,
        name="barrel_shell",
    )
    handle.visual(
        mesh_from_cadquery(_build_top_channel_visual(), "top_channel"),
        material=GRAY,
        name="top_channel",
    )

    # TPR ribs: raised rubber rings around the barrel grip zone
    rib_spacing = (RIB_ZONE_END - RIB_ZONE_START) / (N_RIBS - 1)
    for i in range(N_RIBS):
        x = RIB_ZONE_START + i * rib_spacing
        handle.visual(
            mesh_from_cadquery(_build_tpr_rib(x), f"tpr_rib_{i}"),
            material=TPR_RUBBER,
            name=f"tpr_rib_{i}",
        )

    # --- Rear end cap (fixed) ---------------------------------------------
    end_cap = model.part("end_cap")
    rear_r = _barrel_radius_at(-HANDLE_LEN / 2.0)
    cap_shape = (
        cq.Workplane("YZ")
        .workplane(offset=-HANDLE_LEN / 2.0)
        .center(0.0, BARREL_CENTER_Z)
        .circle(rear_r * 1.02)
        .extrude(-0.012)
    )
    end_cap.visual(
        mesh_from_cadquery(cap_shape, "end_cap"),
        material=DARK_GRAY,
        name="end_cap_body",
    )

    # --- Blade carrier (prismatic slide) ----------------------------------
    carrier = model.part("blade_carrier")

    z_blade_top = HANDLE_H - 0.0035
    nose_x = HANDLE_LEN / 2.0
    blade_rear_x = nose_x - (BLADE_LEN - BLADE_EXPOSED_REST)

    blade = _build_blade_shape().translate((blade_rear_x, 0.0, z_blade_top))
    blade_tip = _build_blade_tip_visual().translate(
        (blade_rear_x, 0.0, z_blade_top)
    )
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
        material=GRAY,
        name="blade_spine",
    )
    carrier.visual(
        mesh_from_cadquery(button, "thumb_button"),
        material=GRAY,
        name="thumb_button",
    )

    # --- Articulations ----------------------------------------------------
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

    # --- Joint type / axis claims ------------------------------------------
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

    # --- Barrel grip geometry claims ---------------------------------------
    barrel_aabb = ctx.part_element_world_aabb(handle, elem="barrel_shell")
    ctx.check(
        "barrel shell has near-circular cross-section (Y and Z extents similar)",
        barrel_aabb is not None
        and abs(
            (barrel_aabb[1][1] - barrel_aabb[0][1])
            - (barrel_aabb[1][2] - barrel_aabb[0][2])
        ) < 0.006,
        details=(
            f"y_extent={None if barrel_aabb is None else barrel_aabb[1][1] - barrel_aabb[0][1]}, "
            f"z_extent={None if barrel_aabb is None else barrel_aabb[1][2] - barrel_aabb[0][2]}"
        ),
    )
    ctx.check(
        "barrel grip diameter is in realistic range (24-34 mm)",
        barrel_aabb is not None
        and 0.024 <= (barrel_aabb[1][1] - barrel_aabb[0][1]) <= 0.034,
        details=(
            f"y_extent={None if barrel_aabb is None else barrel_aabb[1][1] - barrel_aabb[0][1]}"
        ),
    )

    # TPR ribs present and extend beyond barrel surface
    for i in range(N_RIBS):
        rib_name = f"tpr_rib_{i}"
        rib_aabb = ctx.part_element_world_aabb(handle, elem=rib_name)
        ctx.check(
            f"{rib_name} present and wraps around barrel",
            rib_aabb is not None
            and barrel_aabb is not None
            and (rib_aabb[1][1] - rib_aabb[0][1])
            >= (barrel_aabb[1][1] - barrel_aabb[0][1]) * 0.90,
            details=(
                f"rib_y={None if rib_aabb is None else rib_aabb[1][1] - rib_aabb[0][1]}"
            ),
        )

    # At least one rib visibly protrudes beyond the barrel in Y
    mid_rib_aabb = ctx.part_element_world_aabb(handle, elem="tpr_rib_3")
    ctx.check(
        "mid-grip TPR rib protrudes beyond barrel Y extent",
        mid_rib_aabb is not None
        and barrel_aabb is not None
        and (mid_rib_aabb[1][1] - mid_rib_aabb[0][1])
        > (barrel_aabb[1][1] - barrel_aabb[0][1]) + 0.0002,
        details=(
            f"rib_y={None if mid_rib_aabb is None else mid_rib_aabb[1][1] - mid_rib_aabb[0][1]}, "
            f"barrel_y={None if barrel_aabb is None else barrel_aabb[1][1] - barrel_aabb[0][1]}"
        ),
    )

    # --- Blade present and functional --------------------------------------
    blade_aabb = ctx.part_element_world_aabb(carrier, elem="blade_steel")
    handle_aabb = ctx.part_world_aabb(handle)
    ctx.check(
        "segmented blade protrudes past the handle nose at rest",
        blade_aabb is not None
        and handle_aabb is not None
        and blade_aabb[1][0] > handle_aabb[1][0] + 0.004,
        details=(
            f"blade_max_x={None if blade_aabb is None else blade_aabb[1][0]}, "
            f"handle_max_x={None if handle_aabb is None else handle_aabb[1][0]}"
        ),
    )
    button_aabb = ctx.part_element_world_aabb(carrier, elem="thumb_button")
    ctx.check(
        "thumb slide button rises above the handle top",
        button_aabb is not None
        and handle_aabb is not None
        and button_aabb[1][2] > handle_aabb[1][2] - 0.001,
        details=(
            f"button_max_z={None if button_aabb is None else button_aabb[1][2]}, "
            f"handle_max_z={None if handle_aabb is None else handle_aabb[1][2]}"
        ),
    )
    ctx.check(
        "blade has real width (cutting edge below the spine)",
        blade_aabb is not None
        and (blade_aabb[1][2] - blade_aabb[0][2]) > 0.012,
        details=(
            f"blade_z_extent="
            f"{None if blade_aabb is None else blade_aabb[1][2] - blade_aabb[0][2]}"
        ),
    )
    tip_aabb = ctx.part_element_world_aabb(carrier, elem="blade_tip")
    ctx.check(
        "dark blade tip is at the very front",
        tip_aabb is not None
        and blade_aabb is not None
        and tip_aabb[1][0] >= blade_aabb[1][0] - 0.001,
        details=f"tip_max_x={None if tip_aabb is None else tip_aabb[1][0]}",
    )

    # --- End cap seated at the rear ----------------------------------------
    ctx.expect_contact(end_cap, handle, name="end cap seated against handle rear")
    cap_aabb = ctx.part_world_aabb(end_cap)
    ctx.check(
        "end cap is at the rear (-X) of the handle",
        cap_aabb is not None
        and handle_aabb is not None
        and cap_aabb[0][0] <= handle_aabb[0][0] + 0.002,
        details=f"cap_min_x={None if cap_aabb is None else cap_aabb[0][0]}",
    )

    # --- Blade carrier nested in channel -----------------------------------
    ctx.allow_overlap(
        carrier, handle,
        elem_a="blade_spine", elem_b="top_channel",
        reason="The blade spine carrier is intentionally captured inside the "
               "handle's top channel rail and slides along it.",
    )
    ctx.allow_overlap(
        carrier, handle,
        elem_a="blade_steel", elem_b="top_channel",
        reason="The blade root rides inside the handle channel groove as the "
               "carrier slides; this nested fit is intentional.",
    )

    ctx.expect_within(
        carrier, handle,
        axes="y",
        inner_elem="blade_spine", outer_elem="top_channel",
        margin=0.0006,
        name="blade spine stays centered in the channel",
    )
    ctx.expect_overlap(
        carrier, handle,
        axes="x",
        elem_a="blade_steel", elem_b="barrel_shell",
        min_overlap=0.020,
        name="blade remains inserted in the handle at rest",
    )

    # --- Extension pushes blade forward ------------------------------------
    rest_tip = ctx.part_element_world_aabb(carrier, elem="blade_tip")
    with ctx.pose({slide: SLIDE_TRAVEL}):
        ext_tip = ctx.part_element_world_aabb(carrier, elem="blade_tip")
        ctx.check(
            "extending the slide pushes the blade tip forward",
            rest_tip is not None
            and ext_tip is not None
            and ext_tip[1][0] > rest_tip[1][0] + 0.030,
            details=(
                f"rest_tip_x={None if rest_tip is None else rest_tip[1][0]}, "
                f"ext_tip_x={None if ext_tip is None else ext_tip[1][0]}"
            ),
        )
        ctx.expect_overlap(
            carrier, handle,
            axes="x",
            elem_a="blade_steel", elem_b="barrel_shell",
            min_overlap=0.005,
            name="blade stays retained in the handle at full extension",
        )

    return ctx.report()
